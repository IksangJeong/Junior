"""
데이터베이스 연습 #324
SQLite CRUD 실습
"""
import sqlite3

def setup_324():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_324 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_324_0', 379.47, 'C'), ('item_324_1', 641.35, 'C'), ('item_324_2', 913.62, 'C'), ('item_324_3', 492.98, 'A'), ('item_324_4', 536.89, 'B'), ('item_324_5', 21.73, 'C'), ('item_324_6', 724.97, 'C'), ('item_324_7', 977.7, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_324 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_324()
    cursor = conn.cursor()

    print(f"DB 연습 #324")
    cursor.execute('SELECT * FROM items_324')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_324 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
