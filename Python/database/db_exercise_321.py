"""
데이터베이스 연습 #321
SQLite CRUD 실습
"""
import sqlite3

def setup_321():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_321 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_321_0', 652.23, 'B'), ('item_321_1', 268.95, 'C'), ('item_321_2', 883.98, 'B'), ('item_321_3', 107.62, 'C'), ('item_321_4', 474.8, 'B'), ('item_321_5', 915.19, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_321 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_321()
    cursor = conn.cursor()

    print(f"DB 연습 #321")
    cursor.execute('SELECT * FROM items_321')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_321 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
