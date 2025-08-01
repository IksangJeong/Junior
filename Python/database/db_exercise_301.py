"""
데이터베이스 연습 #301
SQLite CRUD 실습
"""
import sqlite3

def setup_301():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_301 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_301_0', 383.06, 'A'), ('item_301_1', 465.07, 'C'), ('item_301_2', 348.9, 'B'), ('item_301_3', 679.11, 'C'), ('item_301_4', 683.94, 'C'), ('item_301_5', 271.72, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_301 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_301()
    cursor = conn.cursor()

    print(f"DB 연습 #301")
    cursor.execute('SELECT * FROM items_301')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_301 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
